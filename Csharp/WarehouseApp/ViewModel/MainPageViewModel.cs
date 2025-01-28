using System.Collections.ObjectModel;
using System.Windows.Input;
using WarehouseApp.Services;

namespace WarehouseApp.ViewModel;

public class MainPageViewModel
{
    public ObservableCollection<CardViewModel> Items { get; }

    public MainPageViewModel()
    {
        Items = new ObservableCollection<CardViewModel>(
            AppState.Instance.Goods
            .Select(goods => new CardViewModel(goods))
        );

        NavigateToOrder = new Command(() => Shell.Current.GoToAsync("//Order"));
    }

    public ICommand NavigateToOrder { get; }
}