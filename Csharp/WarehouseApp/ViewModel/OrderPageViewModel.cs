using System.Collections.ObjectModel;
using WarehouseApp.Services;

namespace WarehouseApp.ViewModel
{
    public class OrderPageViewModel
    {
        public ObservableCollection<OrderCardViewModel> Items { get; }

        public OrderPageViewModel()
        {
            Items = new ObservableCollection<OrderCardViewModel>(
                AppState.Instance.Goods
                .Where(goods => goods.Quantity > 0)
                .Select(goods => new OrderCardViewModel(goods))
            );
        }

        public void RefreshItems()
        {
            Items.Clear();

            var updatedItems = AppState.Instance.Goods
                .Where(goods => goods.Quantity > 0)
                .Select(goods => new OrderCardViewModel(goods));

            foreach (var item in updatedItems)
            {
                Items.Add(item); 
            }
        }
    }
}
