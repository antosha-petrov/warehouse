using System.Windows.Input;

namespace WarehouseApp.ViewModel
{
    public class CartListItemViewModel : BindableObject
    {
        private int _quantity;

        public string Name { get; set; }

        public int Quantity
        {
            get => _quantity;
            set
            {
                _quantity = value;
                OnPropertyChanged();
            }
        }

        public ICommand IncrementCommand { get; }
        public ICommand DecrementCommand { get; }

        public CartListItemViewModel(string name)
        {
            Name = name;
            Quantity = 0;

            IncrementCommand = new Command(() => Quantity++);
            DecrementCommand = new Command(() =>
            {
                if (Quantity > 0)
                    Quantity--;
            });
        }
    }
}
