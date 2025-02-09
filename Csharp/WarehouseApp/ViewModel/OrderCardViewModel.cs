using System.Windows.Input;
using Models;
using WarehouseApp.ViewModel;

public class OrderCardViewModel : ViewModelBase
{
    private readonly OrderItem linkedComponent;

    public int Quantity
    {
        get => linkedComponent.Quantity;
        set
        {
            linkedComponent.Quantity = value;
            OnPropertyChanged();
        }
    }

    public string ImageSource => linkedComponent.Goods.ImageSource;

    public string Title => linkedComponent.Goods.Name;

    public OrderCardViewModel(OrderItem linkedComponent)
    {
        this.linkedComponent = linkedComponent;

        PlusCommand = new Command(() => Quantity++);
        MinusCommand = new Command(() =>
        {
            if (Quantity > 0)
                Quantity--;
        });
    }

    public ICommand PlusCommand { get; }

    public ICommand MinusCommand { get; }
}